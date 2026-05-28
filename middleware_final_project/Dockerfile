FROM python:3.10-slim

WORKDIR /app

COPY middleware/ /app/middleware/
COPY config/ /app/config/
COPY examples/ /app/examples/
COPY tools/ /app/tools/

RUN mkdir -p /app/logs /app/out /app/out_mirror

EXPOSE 1102/tcp

CMD ["python", "-m", "middleware.app", "--config", "config/default_config.json"]