// Generated by CoffeeScript 1.7.1
(function() {
  var logging,
    __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

  logging = (function() {
    function logging(physics) {
      this.getLogfile = __bind(this.getLogfile, this);
      this.logNewPosture = __bind(this.logNewPosture, this);
      this.logTrajectoryData = __bind(this.logTrajectoryData, this);
      this.errorHandler = __bind(this.errorHandler, this);
      this.toggleRecorder = __bind(this.toggleRecorder, this);
      this.physics = physics;
      this.recordTrajectory = false;
      this.startLog = true;
      this.logged_data = [];
      this.fileSystemSize = 100;
      this.fileName = "trace.bin";
      window.requestFileSystem = window.requestFileSystem || window.webkitRequestFileSystem;
    }

    logging.prototype.toggleRecorder = function() {
      this.recordTrajectory = !this.recordTrajectory;
      return this.startLog = this.recordTrajectory;
    };

    logging.prototype.errorHandler = function(e) {
      var msg;
      msg = "";
      switch (e.code) {
        case FileError.QUOTA_EXCEEDED_ERR:
          msg = "QUOTA_EXCEEDED_ERR";
          break;
        case FileError.NOT_FOUND_ERR:
          msg = "NOT_FOUND_ERR";
          break;
        case FileError.SECURITY_ERR:
          msg = "SECURITY_ERR";
          break;
        case FileError.INVALID_MODIFICATION_ERR:
          msg = "INVALID_MODIFICATION_ERR";
          break;
        case FileError.INVALID_STATE_ERR:
          msg = "INVALID_STATE_ERR";
          break;
        default:
          msg = "Unknown Error";
      }
      return console.log("Error: " + msg);
    };

    logging.prototype.logTrajectoryData = function() {
      var onInitFs, parent;
      if (this.recordTrajectory) {
        parent = this;
        onInitFs = function(fs) {
          if (parent.startLog) {
            fs.root.getFile(parent.fileName, {
              create: false
            }, (function(fileEntry) {
              return fileEntry.remove((function() {
                return parent.startLog = false;
              }), parent.errorHandler);
            }), parent.errorHandler);
          }
          return fs.root.getFile(parent.fileName, {
            create: true
          }, (function(fileEntry) {
            parent.logfileURL = fileEntry.toURL();
            return fileEntry.createWriter((function(fileWriter) {
              var bb, buffer, floatView;
              fileWriter.onerror = function(e) {
                return console.log("Write failed: " + e.toString());
              };
              fileWriter.seek(fileWriter.length);
              buffer = new ArrayBuffer(20);
              floatView = new Float32Array(buffer);
              floatView[0] = -this.physics.body.GetAngle();
              floatView[1] = -this.physics.upper_joint.GetJointAngle();
              floatView[2] = -this.physics.lower_joint.GetJointAngle();
              floatView[3] = this.physics.upper_joint.motor_control;
              floatView[4] = this.physics.lower_joint.motor_control;
              bb = new Blob([buffer], {
                type: "application/octet-stream"
              });
              return fileWriter.write(bb);
            }), parent.errorHandler);
          }), parent.errorHandler);
        };
        return window.requestFileSystem(TEMPORARY, parent.fileSystemSize * 1024 * 1024, onInitFs, this.errorHandler);
      }
    };

    logging.prototype.logNewPosture = function() {
      var onInitFs, parent;
      if (this.recordTrajectory) {
        parent = this;
        onInitFs = function(fs) {
          return fs.root.getFile(parent.fileName, {
            create: true
          }, (function(fileEntry) {
            parent.logfileURL = fileEntry.toURL();
            return fileEntry.createWriter((function(fileWriter) {
              var bb, buffer, floatView;
              fileWriter.onerror = function(e) {
                return console.log("Write failed: " + e.toString());
              };
              fileWriter.seek(fileWriter.length - 20);
              buffer = new ArrayBuffer(20);
              floatView = new Float32Array(buffer);
              floatView[0] = Infinity;
              floatView[1] = Infinity;
              floatView[2] = Infinity;
              floatView[3] = Infinity;
              floatView[4] = Infinity;
              bb = new Blob([buffer], {
                type: "application/octet-stream"
              });
              return fileWriter.write(bb);
            }), parent.errorHandler);
          }), parent.errorHandler);
        };
        return window.requestFileSystem(TEMPORARY, parent.fileSystemSize * 1024 * 1024, onInitFs, this.errorHandler);
      }
    };

    logging.prototype.getLogfile = function() {
      var onInitFs, parent;
      parent = this;
      onInitFs = function(fs) {
        return fs.root.getFile(parent.fileName, {
          create: false
        }, (function(fileEntry) {
          console.log("downloading from " + fileEntry.toURL());
          return fileEntry.file(function(file) {
            return saveAs(file, parent.fileName);
          }, parent.errorHandler);
        }), parent.errorHandler);
      };
      return window.requestFileSystem(TEMPORARY, parent.fileSystemSize * 1024 * 1024, onInitFs, this.errorHandler);
    };

    return logging;

  })();

  window.simni.Logging = logging;

}).call(this);

//# sourceMappingURL=logging.map
