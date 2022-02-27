FROM python:3

ENV PYTHONBUFFERED=1

ADD requirements.txt /
ADD *.py /
ADD *.sh /

RUN pip3 install -r requirements.txt
RUN main.sh
