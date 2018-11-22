python setup.py sdist bdist_wheel
python -m twine upload --repository-url https://test.pypi.org/legacy/ dist/*
REM python -m twine upload --repository-url https://upload.pypi.org/legacy/ dist/*

