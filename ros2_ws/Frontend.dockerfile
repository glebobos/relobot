FROM nginx:alpine

COPY src/web_server/frontend/nginx.conf /etc/nginx/conf.d/default.conf
COPY src/web_server/frontend/index.html /usr/share/nginx/html/index.html
COPY src/web_server/frontend/styles.css /usr/share/nginx/html/styles.css
COPY src/web_server/frontend/robot-control.js /usr/share/nginx/html/robot-control.js
COPY src/web_server/frontend/map-viewer.js /usr/share/nginx/html/map-viewer.js
