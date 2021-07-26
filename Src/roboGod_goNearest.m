function [move,mem] = roboGod_goNearest(env,mem)
destination_point = Go_Nearest_Point_Smartly(env);
move = Calculate_Step(destination_point);
move = Correcting_Direction(move);


    % Returns the nearest fuels point smartly
    function destination_point = Go_Nearest_Point_Smartly(temp_env)
        [fuel_op_distance,fuel_op_index] = Nearest_Fuel(temp_env.info.opPos);
        fuels_number = sum(temp_env.fuels.fExist);
        while (fuels_number > 0)
            [fuel_me_distance,fuel_me_index] = Nearest_Fuel(temp_env.info.myPos);
            if fuel_me_distance < (fuel_op_distance + Find_Distance(temp_env.fuels.fPos(fuel_op_index,:),temp_env.fuels.fPos(fuel_me_index,:)))   %%להוסיף בדיקת כדאיות שמתחשבת בכמות הדלקים ובמרחק
                destination_point = temp_env.fuels.fPos(fuel_me_index,:);
                return;
            else
                temp_env.fuels.fExist(fuel_me_index)=0;
                fuels_number = sum(temp_env.fuels.fExist);
            end
        end
        destination_point = env.info.myPos;
        
        % Returns the distance from the nearest fuel point and its index
        function [nearest_fuel_distance,nearest_fuel_index] = Nearest_Fuel(current_point)
            nearest_fuel_distance = sqrt(temp_env.basic.walls(2,3)^2 + temp_env.basic.walls(2,4)^2);  % default value (maximum distance)
            nearest_fuel_index = 0;
            for i = 1:temp_env.fuels.nFuel
                if ((temp_env.fuels.fExist(i) == 1) && (Find_Distance(current_point,temp_env.fuels.fPos(i,:)) < (nearest_fuel_distance)))
                    nearest_fuel_index = i;
                    nearest_fuel_distance = Find_Distance(current_point,temp_env.fuels.fPos(i,:));
                end
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