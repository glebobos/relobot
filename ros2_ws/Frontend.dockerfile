FROM nginx:alpine

# Install Node.js and npm
RUN apk add --no-cache nodejs npm

# Set working directory
WORKDIR /app

# The entrypoint script will be mounted/copied here
COPY src/web_server/frontend/start_frontend.sh /start_frontend.sh
RUN chmod +x /start_frontend.sh

CMD ["/start_frontend.sh"]
