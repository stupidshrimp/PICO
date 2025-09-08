(function(factory){
  if (typeof module === 'object' && typeof module.exports === 'object') {
    module.exports = factory(require('leaflet'));
  } else if (typeof define === 'function' && define.amd) {
    define(['leaflet'], factory);
  } else {
    factory(L);
  }
}(function(L){
  if (!L) { throw new Error('Leaflet is required'); }

  L.TileLayer.MBTiles = L.TileLayer.extend({
    initialize: function(url, options){
      L.TileLayer.prototype.initialize.call(this, url, options);
      this._url = url;
      this._db = null;
      this._initDatabase();
    },

    _initDatabase: function(){
      var self = this;
      if (typeof initSqlJs === 'undefined') {
        console.error('sql.js not loaded');
        return;
      }
      initSqlJs({ locateFile: function(file){ return file; } }).then(function(SQL){
        fetch(self._url).then(function(res){ return res.arrayBuffer(); }).then(function(buf){
          self._db = new SQL.Database(new Uint8Array(buf));
          self._ready = true;
          self.fire('databaseloaded');
        }).catch(function(err){
          console.error('Failed to load MBTiles', err);
        });
      });
    },

    createTile: function(coords, done){
      var tile = document.createElement('img');
      var self = this;
      if (!this._ready) {
        this.once('databaseloaded', function(){
          self._setTileSrc(tile, coords, done);
        });
      } else {
        self._setTileSrc(tile, coords, done);
      }
      return tile;
    },

    _setTileSrc: function(tile, coords, done){
      try {
        var z = coords.z;
        var x = coords.x;
        var y = (1 << z) - 1 - coords.y;
        var stmt = this._db.prepare('SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?');
        stmt.bind([z, x, y]);
        if (stmt.step()) {
          var row = stmt.getAsObject();
          var data = row.tile_data;
          var blob = new Blob([data], { type: 'image/png' });
          tile.src = URL.createObjectURL(blob);
          done(null, tile);
        } else {
          done(new Error('Tile not found'), tile);
        }
        stmt.free();
      } catch (err) {
        done(err, tile);
      }
    }
  });

  L.tileLayer.mbTiles = function(url, options){
    return new L.TileLayer.MBTiles(url, options);
  };

  return L.TileLayer.MBTiles;
}));
