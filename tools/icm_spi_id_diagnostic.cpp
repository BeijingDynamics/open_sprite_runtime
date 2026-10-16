// Read-only WHO_AM_I sweep. No CAN, GPIO, reset or sensor configuration writes.
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <cstring>
#include <initializer_list>
#include <csignal>
#include <cstdlib>

static volatile std::sig_atomic_t stopped = 0;
static void stop(int) { stopped = 1; }

static void check(bool ok) { if (!ok) throw std::runtime_error("SPI ioctl failed"); }
int main(int argc, char** argv) {
    const bool loopback = argc == 2 && std::strcmp(argv[1], "--external-loopback-module-disconnected") == 0;
    const bool pattern = argc == 3 && std::strcmp(argv[1], "--continuous-aa-500khz") == 0;
    const bool continuous = argc == 3 && (pattern || std::strcmp(argv[1], "--continuous-id-100khz") == 0 || std::strcmp(argv[1], "--continuous-id-500khz") == 0)
        && (std::strcmp(argv[2], "0") == 0 || std::strcmp(argv[2], "3") == 0);
    if (argc != 1 && !loopback && !continuous) { fprintf(stderr, "Invalid arguments\n"); return 1; }
    const unsigned selected_mode = continuous && argv[2][0] == '0' ? 0 : 3;
    const unsigned selected_speed = continuous && (pattern || std::strcmp(argv[1], "--continuous-id-500khz") == 0) ? 500000 : 100000;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    setvbuf(stdout, nullptr, _IOLBF, 0);
    const char* device = std::getenv("SPRITE_SPI_DEVICE");
    if (!device) device = "/dev/spidev0.0";
    if (std::strcmp(device, "/dev/spidev0.0") != 0 && std::strcmp(device, "/dev/spidev1.0") != 0) {
        fprintf(stderr, "Unsupported SPI device\n"); return 1;
    }
    const int fd = open(device, O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("open"); return 1; }
    if (flock(fd, LOCK_EX | LOCK_NB)) { perror("flock"); close(fd); return 1; }
    uint32_t old_mode = 0, old_speed = 0;
    uint8_t old_bits = 0, old_lsb = 0;
    try {
        check(ioctl(fd, SPI_IOC_RD_MODE32, &old_mode) == 0);
        check(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &old_speed) == 0);
        check(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &old_bits) == 0);
        check(ioctl(fd, SPI_IOC_RD_LSB_FIRST, &old_lsb) == 0);
    } catch (...) { close(fd); return 1; }
    int result = 0;
    uint64_t matches = 0, total = 0;
    try {
        uint8_t bits = 8, lsb = 0;
        check(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == 0);
        check(ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb) == 0);
        if (continuous) printf("CONTINUOUS %s CS0 mode=%u requested=%uHz TX=%s; sleep 10ms after each transfer; Ctrl+C stops; NO_CAN\n", device, selected_mode, selected_speed, pattern ? "AA,AA (RX ignored)" : "F2,00 expected RX[1]=E9");
        for (uint32_t mode : {0u, 3u}) {
            if (stopped || (continuous && mode != selected_mode)) continue;
            check(ioctl(fd, SPI_IOC_WR_MODE32, &mode) == 0);
            for (uint32_t speed : {100000u, 500000u, 1000000u}) {
                if (stopped || (continuous && speed != selected_speed)) continue;
                check(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == 0);
                uint32_t read_mode = 0, read_speed = 0;
                check(ioctl(fd, SPI_IOC_RD_MODE32, &read_mode) == 0);
                check(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &read_speed) == 0);
                for (uint64_t i = 0; !stopped && (continuous || i < 5); ++i) {
                    uint8_t tx[16] = {0xf2, 0x00}, rx[16];
                    if (pattern) { tx[0] = 0xaa; tx[1] = 0xaa; }
                    std::memset(rx, 0xcc, sizeof(rx));
                    const unsigned length = loopback ? 16 : 2;
                    if (loopback) {
                        const uint8_t pattern[16] = {0x00,0xff,0xaa,0x55,0x01,0x80,0x12,0x34,
                                                     0x56,0x78,0x9a,0xbc,0xde,0xf0,0x0f,0xa5};
                        for (unsigned j=0;j<16;++j) tx[j]=pattern[(j+i)%16];
                    }
                    spi_ioc_transfer x{};
                    x.tx_buf = reinterpret_cast<uintptr_t>(tx);
                    x.rx_buf = reinterpret_cast<uintptr_t>(rx);
                    x.len = length; x.speed_hz = speed; x.bits_per_word = 8;
                    const int n = ioctl(fd, SPI_IOC_MESSAGE(1), &x);
                    check(n == int(length));
                    const bool matched=pattern ? true : (loopback ? std::memcmp(tx,rx,length)==0 : rx[1]==0xe9);
                    matches += matched;
                    ++total;
                    if (!continuous || i % 100 == 0) {
                    printf("mode=%u requested_hz=%u readback_hz=%u sample=%llu bytes=%d match=%d tx=",
                           read_mode, speed, read_speed, static_cast<unsigned long long>(i), n, matched);
                    for(unsigned j=0;j<length;++j) printf("%02X",tx[j]);
                    printf(" rx=");
                    for(unsigned j=0;j<length;++j) printf("%02X",rx[j]);
                    printf("\n");
                    }
                    usleep(continuous ? 10000 : 20000);
                }
            }
        }
        printf("%s_MATCHES=%llu/%llu; speed readback is not a measured wire clock; NO_CAN\n", pattern ? "TX_COMPLETED_RX_NOT_VALIDATED" : (loopback ? "EXTERNAL_LOOPBACK" : "ID"), static_cast<unsigned long long>(matches), static_cast<unsigned long long>(total));
        if (matches != total) result = 2;
    } catch (const std::exception& e) { fprintf(stderr, "%s\n", e.what()); result = 1; }
    bool restored = ioctl(fd, SPI_IOC_WR_MODE32, &old_mode) == 0;
    restored &= ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &old_bits) == 0;
    restored &= ioctl(fd, SPI_IOC_WR_LSB_FIRST, &old_lsb) == 0;
    restored &= ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &old_speed) == 0;
    printf("SPI_CONFIGURATION_RESTORED=%s\n", restored ? "true" : "false");
    close(fd);
    return restored ? result : 1;
}
