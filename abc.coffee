#methods for exploring postures, ABC style

class abc
  constructor: ->
    @postures = []  #graph

  searchSubarray: (sub, array, eps=0.008) =>
    found = []
    #return index if subarray found
    for i in [0..array.length-sub.length] by 1
      for j in [0..sub.length-1] by 1
        if Math.abs(sub[j][0] - array[i+j][0]) > eps or Math.abs(sub[j][1] - array[i+j][1]) > eps or Math.abs(sub[j][2] - array[i+j][2]) > eps
          break
      if j == sub.length
        found.push i
        i = _i = i+sub.length
    
    if found.length is 0
      return false
    else
      return found

  e1 = 0.02
  e2 = 0.04
  e1_body = 0.1
  e2_body = 0.15
  fp_flag = false
  MAX_UNIX_TIME = 1924988399 #31/12/2030 23:59:59
  time = time2 = MAX_UNIX_TIME
  trajectory = []   #last n state points
  detectAttractor: (body, upper_joint, lower_joint) =>
    #detect if current csl mode found a posture
    #TODO:
    #- gb sometimes is to small to correctly prepare next contraction
    #- some very slow contraction periodic attractors might not be detected
    #- only detect posture after a few seconds have passed after switching mode
    #  (sometimes very quickly same pose is detected again)
    #- do something if csl hits limit (switch to r of same direction and set bias to current csl value/large value)
    #- act on detection event following some strategy:
    #  randomly select next mode or try new mode that we have not seen yet or change joint csl that was
    #  not changed before
    #- build graph a found postures

    #get angle velocities
    dp_body = Math.abs body.GetAngularVelocity()     #dp means ∆φ
    dp_hip = Math.abs upper_joint.GetJointSpeed()
    dp_knee = Math.abs lower_joint.GetJointSpeed()
    
    ###
    #find only fixpoints, find 4 seconds of slow speed
    if dp_body < e1_body and dp_hip < e1 and dp_knee < e1
      fp_flag = true
  
      if time2 is MAX_UNIX_TIME   #starting to check if we have a fixpoint
        time2 = new Date().getTime()

    if dp_body > e2_body or dp_hip > e2 or dp_knee > e2
      fp_flag = false
      time2 = MAX_UNIX_TIME

    if (new Date().getTime() - time2) > 4000 and fp_flag is true
      console.log("found fixpoint")
      fp_flag = false
      time2 = MAX_UNIX_TIME
    ###

    #find attractors, take a sample of trajectory and try to find it multiple times in the
    #past trajectory (with epsilon), hence (quasi)periodic behaviour
    if trajectory.length==3000   #corresponds to max periode duration that can be detected
      trajectory.shift()

    trajectory.push [dp_body, dp_hip, dp_knee]

    if trajectory.length > 200 and (new Date().getTime() - time) > 2000
      #take last 40 points
      last = trajectory.slice(-40)
      d = @searchSubarray last, trajectory
      #console.log(d)

      if d.length > 3    #need to find sample more than once to be periodic
        posture = [body.GetAngle(), upper_joint.GetJointAngle(), lower_joint.GetJointAngle()]
        console.log("found pose/attractor:" + posture)
        
        #save posture
        @savePosture posture

        #get rid of saved trajectory
        trajectory = []

      time = new Date().getTime()

  savePosture: (posture) =>
    #expects posture = [body angle, hip angle, knee angle]
    if not @searchSubarray [posture], @postures, 0.1
      #we dont have something close to this posture yet, add it
      @postures.push(posture)

  limitCSL: (joints) =>
    for j in joints
      if j.csl_active and j.motor_control > 20
        #csl goes against limit, set to release mode with current csl value as bias
        1

  update: (body, upper_joint, lower_joint) =>
    @detectAttractor body, upper_joint, lower_joint
    @limitCSL [upper_joint, lower_joint]
