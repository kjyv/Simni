
class logging
  constructor: (physics) ->
    @physics = physics
    @recordTrajectory = false  #if we log state data for saving later
    @startLog = true      #if we start a new log
    @logged_data = []     #where we log our data
    @fileSystemSize = 100 #mb
    @fileName = "trace.bin"


    window.requestFileSystem = window.requestFileSystem || window.webkitRequestFileSystem

  toggleRecorder: =>
    @recordTrajectory = !@recordTrajectory
    @startLog = @recordTrajectory

  errorHandler: (e) =>
    msg = ""
    switch e.code
      when FileError.QUOTA_EXCEEDED_ERR
        msg = "QUOTA_EXCEEDED_ERR"
      when FileError.NOT_FOUND_ERR
        msg = "NOT_FOUND_ERR"
      when FileError.SECURITY_ERR
        msg = "SECURITY_ERR"
      when FileError.INVALID_MODIFICATION_ERR
        msg = "INVALID_MODIFICATION_ERR"
      when FileError.INVALID_STATE_ERR
        msg = "INVALID_STATE_ERR"
      else
        msg = "Unknown Error"
    console.log "Error: " + msg


  logTrajectoryData: =>
    #called repeatedly; save trajectory into sandbox-local file
    if @recordTrajectory
      parent = @
      onInitFs = (fs) ->
        if parent.startLog
          fs.root.getFile parent.fileName, create: false, ((fileEntry) ->
            fileEntry.remove (->
              parent.startLog = false
            ), parent.errorHandler
          ), parent.errorHandler

        fs.root.getFile parent.fileName, create: true, ((fileEntry) ->
          parent.logfileURL = fileEntry.toURL()

          # Create a FileWriter object for our FileEntry (log.txt)
          fileEntry.createWriter ((fileWriter) ->
            fileWriter.onerror = (e) ->
              console.log "Write failed: " + e.toString()

            fileWriter.seek(fileWriter.length)

            # Create a new Blob and write it to log.txt
            buffer = new ArrayBuffer(20) #4 bytes for 32 bit float * 5 = 20
            floatView = new Float32Array(buffer)
            floatView[0] = -@physics.body.GetAngle()
            floatView[1] = -@physics.upper_joint.GetJointAngle()
            floatView[2] = -@physics.lower_joint.GetJointAngle()
            floatView[3] = @physics.upper_joint.motor_control
            floatView[4] = @physics.lower_joint.motor_control
            bb = new Blob([buffer], { type: "application/octet-stream" })

            fileWriter.write bb
          ), parent.errorHandler
        ), parent.errorHandler
      window.requestFileSystem TEMPORARY, parent.fileSystemSize * 1024 * 1024, onInitFs, @errorHandler #100MB

  logNewPosture: =>
    if @recordTrajectory
      parent = @
      onInitFs = (fs) ->
        fs.root.getFile parent.fileName, create: true, ((fileEntry) ->
          parent.logfileURL = fileEntry.toURL()

          # Create a FileWriter object for our FileEntry (log.txt)
          fileEntry.createWriter ((fileWriter) ->
            fileWriter.onerror = (e) ->
              console.log "Write failed: " + e.toString()

            #overwrite last line (number of bytes!) so we don't introduce a time shift
            fileWriter.seek(fileWriter.length-20)

            # Create a new Blob and write it to log.txt
            buffer = new ArrayBuffer(20) #4 bytes for 32 bit float * 5 = 20
            floatView = new Float32Array(buffer)
            floatView[0] = Infinity
            floatView[1] = Infinity
            floatView[2] = Infinity
            floatView[3] = Infinity
            floatView[4] = Infinity
            bb = new Blob([buffer], { type: "application/octet-stream" })

            fileWriter.write bb
          ), parent.errorHandler
        ), parent.errorHandler

      window.requestFileSystem TEMPORARY, parent.fileSystemSize * 1024 * 1024, onInitFs, @errorHandler #100MB

  getLogfile: =>
    #download log file from sandbox
    parent = @
    onInitFs = (fs) ->
      fs.root.getFile parent.fileName, create: false, ((fileEntry) ->
        console.log("downloading from " + fileEntry.toURL())
        fileEntry.file (file) ->
          saveAs file, parent.fileName
        , parent.errorHandler
      ), parent.errorHandler
    window.requestFileSystem TEMPORARY, parent.fileSystemSize * 1024 * 1024, onInitFs, @errorHandler

window.simni.Logging = logging
