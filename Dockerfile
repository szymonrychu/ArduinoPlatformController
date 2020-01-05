FROM alpine:latest

COPY ds4controller.py /usr/bin/ds4controller
COPY requirements.txt /

RUN set -xe; \
    chmod +x /usr/bin/ds4controller;\
    apk --no-cache --update add \
        python3;\
    pip3 install --upgrade pip;\
    pip install -r /requirements.txt;\
    rm /requirements.txt

CMD [ "/usr/bin/env", "python3", "/usr/bin/ds4controller" ]