# Map assets

This directory holds third-party files required to display the map completely offline.
Copy the following files into this directory:

- `maplibre-gl.js` and `maplibre-gl.css` – MapLibre GL library
- `pmtiles.js` – Protomaps PMTiles reader
- `map1.pmtiles` – raster tiles bundle

The application uses the `map1.pmtiles` database exclusively and will not fetch tiles
from the internet. If the file is missing or unreadable the GPS view will stay blank
and display an error message instructing you to copy the archive into this directory.
