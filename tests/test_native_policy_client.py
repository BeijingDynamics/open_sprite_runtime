import unittest

from open_sprite_runtime.native_policy_client import _capture_runtime_errors


class RuntimeErrorCaptureTests(unittest.TestCase):
    def test_runtime_error_is_recorded_without_escaping(self) -> None:
        errors: list[str] = []

        with _capture_runtime_errors(errors):
            raise RuntimeError("watchdog test trip")

        self.assertEqual(errors, ["RuntimeError: watchdog test trip"])

    def test_clean_exit_does_not_add_an_error(self) -> None:
        errors: list[str] = []

        with _capture_runtime_errors(errors):
            pass

        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
