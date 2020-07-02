classdef Robot<handle
    properties
      x;
      y;
      orientation;
      forward_noise;
      turn_noise;
      sense_noise;
    end
    
    methods
        function obj = Robot()
            obj.x = 0;
            obj.y = 0;
            obj.orientation = 0;
            obj.forward_noise = 0;
            obj.turn_noise = 0;
            obj.sense_noise = 0;
        end
        function set(obj, new_x, new_y, new_orientation)
            obj.x = new_x;
            obj.y = new_y;
            obj.orientation = new_orientation;
        end
        function set_noise(obj, new_f_noise, new_t_noise, new_s_noise)
            % makes it possible to change the noise parameters
            obj.forward_noise = new_f_noise;
            obj.turn_noise    = new_t_noise;
            obj.sense_noise   = new_s_noise;
        end
        function Z = sense(obj,landmarks)
            Z = zeros(1,length(landmarks(:,1)));
            for i = 1:(length(landmarks(:,1)))
                dist = sqrt((obj.x - landmarks(i,1)).^2 + (obj.y - landmarks(i,2)).^2);
                dist = dist + normrnd(0.0, obj.sense_noise);
                Z(i) = dist;
            end
        end
        function res = move(obj, angular, forward, dt)
            % turn, and add randomness to the turning command
        	orientation1 = obj.orientation + angular*dt + normrnd(0.0, obj.turn_noise);
	        dist = forward*dt + normrnd(0.0, obj.forward_noise);
	        y1 = obj.y + dist*cos(orientation1);
            x1 = obj.x + dist*sin(orientation1);
            % set particle
	        res = Robot();
        	res.set(x1, y1, orientation1)
	        res.set_noise(obj.forward_noise, obj.turn_noise, obj.sense_noise)
        end
        function g = Gaussian(obj, mu, sigma, x)
            % calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
            g = exp(-((mu - x).^2) / (sigma.^2) / 2.0) / sqrt(2.0 * pi * (sigma.^2));
        end
        function prob = measurement_prob(obj, measurement, landmarks)
            % calculates how likely a measurement should be
            prob = 1.0;
            for i = 1:length(landmarks(:,1))
                dist = sqrt((obj.x - landmarks(i,1)).^2 + (obj.y - landmarks(i,2)).^2);
                prob = prob * obj.Gaussian(dist, obj.sense_noise, measurement(i));
            end
        end
    end
end

