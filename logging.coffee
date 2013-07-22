
class logging
  constructor: (physics) ->
    @physics = physics
    @recordTrajectory = false  #if we log state data for saving later
    @startLog = true      #if we start a new log
    @logged_data = []     #where we log our data

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
          fs.root.getFile "log.txt", create: false, ((fileEntry) ->
            fileEntry.remove (->
              parent.startLog = false
            ), parent.errorHandler
          ), parent.errorHandler

        fs.root.getFile "log.txt", create: true, ((fileEntry) ->
          parent.logfileURL = fileEntry.toURL()

          # Create a FileWriter object for our FileEntry (log.txt)
          fileEntry.createWriter ((fileWriter) ->
            fileWriter.onerror = (e) ->
              console.log "Write failed: " + e.toString()

            fileWriter.seek(fileWriter.length)

            # Create a new Blob and write it to log.txt
            # TODO: use direct byte data for this
            bb = new Blob([-@physics.body.GetAngle() + " " + -@physics.upper_joint.GetJointAngle() + " " + -@physics.lower_joint.GetJointAngle()+ " " + @physics.upper_joint.motor_control + " " + @physics.lower_joint.motor_control], { type: "application/octet-stream" })

            fileWriter.write bb
          ), parent.errorHandler
        ), parent.errorHandler
      window.requestFileSystem TEMPORARY, 5 * 1024 * 1024, onInitFs, @errorHandler #5MB

  getLogfile: =>
    #download log file from sandbox (and disable logging)
    if @recordTrajectory
      $("#start_log").click()

    parent = @
    onInitFs = (fs) ->
      fs.root.getFile "log.txt", create: true, ((fileEntry) ->
        console.log("downloading from " + fileEntry.toURL())
        fileEntry.file (file) ->
          saveAs file, "log.txt"
        , parent.errorHandler
      ), parent.errorHandler
    window.requestFileSystem TEMPORARY, 5 * 1024 * 1024, onInitFs, @errorHandler #5MB

window.simni.Logging = logging
