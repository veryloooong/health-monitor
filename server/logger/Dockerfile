FROM python:3.9-slim

# Set working directory
WORKDIR /app

# Install MQTT library
RUN pip install --no-cache-dir paho-mqtt

# Copy the script into the container
COPY logger.py .

# Run the script
CMD ["python", "-u", "logger.py"]
