

def create_load_tests(filepath):
    from pathlib import Path
    start_dir = Path(filepath).parent
    def load_tests(loader, standard_tests, pattern):
        pattern = pattern or "test_*.py"
        package_tests = loader.discover(start_dir=start_dir, pattern=pattern)
        standard_tests.addTests(package_tests)
        return standard_tests
    return load_tests


load_tests = create_load_tests(__file__)
