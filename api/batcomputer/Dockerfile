FROM python:3

ENV PYTHONBUFFERED=1

ADD requirements.txt /
ADD *.py /

RUN pip3 install -r requirements.txt
RUN uvicorn api:app --reload --host "0.0.0.0"
