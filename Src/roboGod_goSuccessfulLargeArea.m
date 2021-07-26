function [move,mem] = roboGod_goSuccessfulLargeArea(env,mem)
[did_find,destination_point] = Check_On_The_Way_Point();
if (did_find == 0)
    [destination_point] = Find_Best_Area_Destination_Point();
end
move = Calculate_Step(destination_point);
move = Correcting_Direction(move);


    % Returns the nearest fuels point smartly
    function [nearest_fuel_distance,nearest_fuel_index] = Nearest_Fuel(current_point)
        nearest_fuel_distance = sqrt(env.basic.walls(2,3)^2 + env.basic.walls(2,4)^2);  % default value (maximum distance)
        nearest_fuel_index = 0;
        for i = 1:env.fuels.nFuel
            if ((env.fuels.fExist(i) == 1) && (Find_Distance(current_point,env.fuels.fPos(i,:)) < (nearest_fuel_distance)))
                nearest_fuel_index = i;
                nearest_fuel_distance = Find_Distance(current_point,env.fuels.fPos(i,:));
            end
        end
    end


    % Fix step to avoid mistakes
    function fixed_move = Correcting_Direction(old_move)
        temp_location = env.info.myPos + old_move;
        fixed_move = old_move;
        i=1;
        flag=0;
        while (i < env.mines.nMine)
            if ((env.mines.mExist(i) == 1) && (Find_Distance(temp_location,env.mines.mPos(i,:)) < (0.64*(env.basic.rMF + env.basic.rRbt))))
                const_angel = arctan_calculation(old_move);
                changing_angel = 0;
                while (changing_angel < (pi/2))
                    fixed_move_clockwise = Adding_Angle_New_Move(const_angel,changing_angel);
                    temp_location = env.info.myPos +  fixed_move_clockwise;
                    for  j = 1:env.mines.nMine
                        if ((env.mines.mExist(j) == 1) && (Find_Distance(temp_location,env.mines.mPos(j,:)) < (0.64*(env.basic.rMF + env.basic.rRbt))))
                            flag=1;
                            break;
                        end
                    end
                    if (flag == 0)
                        fixed_move = fixed_move_clockwise;
                        return;
                    else
                        flag = 0;
                    end
                    fixed_move_anticlockwise = Adding_Angle_New_Move(const_angel,-1*changing_angel);
                    temp_location = env.info.myPos +  fixed_move_anticlockwise;
                    for  j = 1:env.mines.nMine
                        if ((env.mines.mExist(j) == 1) && (Find_Distance(temp_location,env.mines.mPos(j,:)) < (0.64*(env.basic.rMF + env.basic.rRbt))))
                            flag=1;
                            break;
                        end
                    end
                    if (flag == 0)
                        fixed_move = fixed_move_anticlockwise;
                        return;
                    else
                        flag = 0;
                    end
                    changing_angel = changing_angel + 0.001;   % Responsible for accuracy
                end
            end
            i=i+1;
        end
        
        % Connecting angles and conversion from polar to cartesian
        function fixed_move = Adding_Angle_New_Move(const_angel,add_angel)
            new_angel = const_angel + add_angel;
            if (new_angel >= 2*pi)
                new_angel = new_angel - 2*pi;
            elseif (new_angel < 0)
                new_angel = new_angel + 2*pi;
            end
            if (new_angel < pi/2)   % I
                fixed_move(1) = env.basic.lmax * cos(new_angel);
                fixed_move(2) = env.basic.lmax * sin(new_angel);
            elseif ((new_angel > pi/2) && (new_angel < pi))   % II
                fixed_move(1) = -1 * env.basic.lmax * cos(pi - new_angel);
                fixed_move(2) = env.basic.lmax * sin (pi - new_angel);
            elseif ((new_angel > pi) && (new_angel < 3/2*pi))  % III
                fixed_move(1) = -1 * env.basic.lmax * cos(new_angel - pi);
                fixed_move(2) = -1 * env.basic.lmax * sin(new_angel - pi);
            elseif (new_angel > 3/2*pi)   % IIII
                fixed_move(1) = env.basic.lmax * cos(2*pi-new_angel);
                fixed_move(2) = -1 * env.basic.lmax * sin(2*pi-new_angel);
            elseif (new_angel == 0)
                fixed_move(1) = env.basic.lmax;
                fixed_move(2) = 0;
            elseif (new_angel == pi/2)
                fixed_move(1) = 0;
                fixed_move(2) = env.basic.lmax;
            elseif (new_angel == pi)
                fixed_move(1) = -1*env.basic.lmax;
                fixed_move(2) = 0;
            elseif (new_angel == 3/2*pi)
                fixed_move(1) = 0;
                fixed_move(2) = -1*env.basic.lmax;
            else
                fixed_move = [0 0];
            end
        end
        
        % Transition from cartesian to polar
        function angle = arctan_calculation(move)
            if ((move(1) > 0) && (move(2) > 0))        % x,y
                angle = atan(move(2)/(move(1)));
            elseif ((move(1) < 0) && (move(2) > 0))    % -x,y
                angle = pi - atan(abs(move(2))/(abs(move(1))));
            elseif ((move(1) < 0) && (move(2) < 0))   % -x,-y
                angle = pi + atan(abs(move(2))/(abs(move(1))));
            elseif ((move(1) > 0) && (move(2) < 0))   % x,-y
                angle = atan(move(2)/(move(1)));
            elseif ((move(1) == 0) && (move(2) > 0))  % pi/2
                angle = pi/2;
            elseif ((move(1) == 0) && (move(2) < 0))  % 3/2*pi;
                angle = 3/2*pi;
            elseif ((move(2) == 0) && (move(1) < 0))  % pi;
                angle = pi;
            else
                angle = 0;
            end
        end
    end


    % Returns a point on the way if there is one
    function [did_find,destination_point] = Check_On_The_Way_Point()
        max_steps_around = 1;     % to cancel enter 0
        for i = 1:env.fuels.nFuel
            if ((env.fuels.fExist(i) == 1) && (Find_Distance(env.info.myPos,env.fuels.fPos(i,:)) < (0.62*(env.basic.rRbt + env.basic.rMF)) + (max_steps_around * env.basic.lmax)))
                destination_point = env.fuels.fPos(i,:);
                did_find = 1;
                return;
            end
        end
        destination_point = env.info.myPos;
        did_find = 0;
    end


    % Find the best area to go to
    function [destination_point] = Find_Best_Area_Destination_Point()
        max_possible_distance_in_map = sqrt(env.basic.walls(2,3)^2 + env.basic.walls(2,4)^2);  % default value for maximum distance
        % Radius calculation
        d_m = 1:env.fuels.nFuel;
        d_o = 1:env.fuels.nFuel;
        R = 1:env.fuels.nFuel;
        for i = 1:env.fuels.nFuel
            d_m(i) = Find_Distance(env.info.myPos,env.fuels.fPos(i,:));
            [fuel_op_distance,fuel_op_index] = Nearest_Fuel(env.info.opPos);
            if ((fuel_op_index == i) || (fuel_op_index == 0))
                d_o(i) = Find_Distance(env.info.opPos,env.fuels.fPos(i,:));
            else
                d_o(i) = (fuel_op_distance + Find_Distance(env.fuels.fPos(fuel_op_index,:),env.fuels.fPos(i,:)));
            end
            R(i) = (d_o(i) - d_m(i))/2;
        end
        
        % Calculates the amount of points within the circle
        number_points_around = 1:env.fuels.nFuel;
        for i = 1:env.fuels.nFuel
            if ((R(i) < 0) || (env.fuels.fExist(i) == 0))
                number_points_around(i) = 0;
                continue;
            elseif (R(i) == 0)      
                number_points_around(i) = 1;
                continue;
            end
            mone = 0;
            for j = 1:env.fuels.nFuel
                if ((Find_Distance(env.fuels.fPos(i,:),env.fuels.fPos(j,:))) <= R(i))
                    mone = mone + 1;
                end
            end
            number_points_around(i) = mone;
        end
        
        % Calculates the region that is the best target
        max_area_points_number = 1;
        max_area_index = 0;
        max_area_min_distance = max_possible_distance_in_map;
        for i = 1:env.fuels.nFuel
            if (number_points_around(i) > max_area_points_number)
                max_area_points_number = number_points_around(i);
                max_area_index = i;
                max_area_min_distance = d_m(i);
            elseif ((number_points_around(i) == max_area_points_number) && (d_m(i) < max_area_min_distance))
                max_area_index = i;
                max_area_min_distance = d_m(i);
            end
        end
        
        % Find the point in the circle closest to my robot and Endpoint testing.
        if (max_area_index == 0)
            destination_point = env.info.myPos;
        else
            min_distance_size = max_possible_distance_in_map;
            min_distance_index = 0;
            for i = 1:env.fuels.nFuel
                if ((env.fuels.fExist(i) == 1) && (Find_Distance(env.fuels.fPos(max_area_index,:),env.fuels.fPos(i,:)) <= R(i)) && (Find_Distance(env.info.myPos,env.fuels.fPos(i,:)) < min_distance_size))
                    min_distance_index = i;
                    min_distance_size = Find_Distance(env.info.myPos,env.fuels.fPos(i,:));
                end
            end
            destination_point = env.fuels.fPos(min_distance_index,:);
        end
    end


    % Finding a distance between two points
    function d = Find_Distance (start_point,end_point)
        es = end_point - start_point;
        d = sqrt(es*es');
    end


    % Calculates the step that needs to go towards a destination point
    function move = Calculate_Step (end_point)
        max_step_size = env.basic.lmax;  % Just to simplify
        start_point = env.info.myPos;    % Just to simplify
        es = end_point - start_point;
        move = max_step_size*es/sqrt(es*es');
    end
end