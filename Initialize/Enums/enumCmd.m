classdef enumCmd < int32
    enumeration
        undefined (0)

        reboot_arm (1)
        emergency_stop (2)
        clear_faults (3)
        stop_action (4)
        pause_action (5)
        resume_action (6)
        
        precomputed_joint_trj (101)
        
        joint_reach (200)
        
        cartesian_reach (300)
        
        tool_reach (400)
        tool_speed (401)
        activate_vacuum(500)
        deactivate_vacuum(501)

        send_to_home(600)
    end
end
