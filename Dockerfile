FROM python:3-alpine

COPY . /code

RUN (cd /code; pip install -r requirements.txt)

ENTRYPOINT [ "/code/main.py" ]
