
# Build stage
FROM node:22-alpine AS build

WORKDIR /app

# Copy package files from the correct location relative to build context
COPY src/web_ui/package*.json ./
RUN npm install

# Copy source files
COPY src/web_ui/ .
RUN npm run build

# Serve stage
FROM nginx:alpine

COPY --from=build /app/dist /usr/share/nginx/html

# Copy custom nginx config
COPY src/web_ui/nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]
