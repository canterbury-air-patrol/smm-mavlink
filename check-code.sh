#!/bin/bash -ex

source venv/bin/activate

pycodestyle --ignore=E501 *.py

pylint *.py

deactivate
