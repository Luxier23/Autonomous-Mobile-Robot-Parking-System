classdef Fake_Wheel < handle
    properties (Access = private)
        dt = 0.0;
        N = 0.0;
        R = 0.0;

        d = 0;
        c = 0;
    end
    
    methods
        function obj = Fake_Wheel(N,R,dt)
            obj.N = N;
            obj.R = R;
            obj.dt = dt; 
        end
        
        function obj = Set_Wheel_Velocity (obj,w) 
            v = w*obj.R;
            obj.d = obj.d + v*obj.dt;
            obj.c = floor((obj.N/(2*pi*obj.R))*obj.d);
        end

        function c = Read_Enconder_Count (obj)
            c = obj.c;
        end
    end
end
