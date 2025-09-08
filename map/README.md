# Map assets

This directory holds third-party files required to display the map completely offline. Copy the following files into this directory:
- `leaflet.js` and `leaflet.css`
- `mbtiles.js`
- `sql-wasm.js` and `sql-wasm.wasm`
- `map1.mbtiles` (the tile database)

The application uses the MBTiles database exclusively and will not fetch tiles from the internet. If these files are missing, the map area will be blank.
