FROM python:3.11-slim

WORKDIR /app

# can-utils provides candump/cansend for debugging inside the container
RUN apt-get update && apt-get install -y --no-install-recommends \
        can-utils \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY conftest.py .
COPY xcp_fd_slave/ ./xcp_fd_slave/
COPY xcp_master_cantest.py .

# The slave needs --network host and --privileged (or --cap-add NET_ADMIN)
# on the host so it can reach the vcan0 interface.
# Example run:
#   docker run --rm --network host --privileged clumsysenpai/xcp_slave
ENTRYPOINT ["python3", "-m", "xcp_fd_slave.main"]
CMD ["--channel", "vcan0", "--log-level", "INFO"]
