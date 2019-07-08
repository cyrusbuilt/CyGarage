# OpenHAB Components

This directory contains components for integrating with OpenHAB2.

## cygarage.items

Edit this file and change each '<your_ip_here>' with the IP of your device. Copy this file to your openhab-conf/items folder.

## cygarage.sitemap

Copy this file to your openhab-conf/sitemaps folder.

## cygarage.rules

Edit this file and replace '<your_ip_here>' with the IP of your device. Copy this file to your openhab-conf/rules folder.

## Dependencies

You will need the following dependent add-ons installed in OpenHAB2 for these components to work correctly:

[HTTP Binding](https://www.openhab.org/addons/bindings/http1/)

[openHAB Cloud Connector](https://www.openhab.org/addons/integrations/openhabcloud/)

- You also need to connect your local OpenHAB server to a cloud instance in order to receive push notifications.
