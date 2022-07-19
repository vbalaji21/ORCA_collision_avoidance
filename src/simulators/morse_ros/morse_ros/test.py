import pymorse
with pymorse.Morse() as sim:
    print(sim.rpc('simulation', 'list_robots') )
    print()
    
    sim.rpc('simulation', 'set_object_position','pr2',[2,2,0]) 
    sim.rpc('simulation', 'details')
    sim.rpc('simulation', 'deactivate', 'HumanSkeleton')
    #sim.rpc('simulation', 'reset_objects') 
    #sim.rpc('simulation', 'set_camarafp_far_clip', 1000)

