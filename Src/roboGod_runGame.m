close all
team1 = struct('name','goNearest','color',rand(1,3),'strategy',@roboGod_goNearest);       
team2 = struct('name','goSuccessfulArea','color',rand(1,3),'strategy',@roboGod_goSuccessfulArea);     
robotgame_main(team1,team2);

close all
team1 = struct('name','goSuccessfulAreaTeam A','color',rand(1,3),'strategy',@roboGod_goSuccessfulAreal);
team2 = struct('name','goSuccessfulLargeArea','color',rand(1,3),'strategy',@roboGod_goSuccessfulLargeArea);  
robotgame_main(team1,team2);

close all
team1 = struct('name','goNearest','color',rand(1,3),'strategy',@roboGod_goNearest);
team2 = struct('name','goSuccessfulLargeArea','color',rand(1,3),'strategy',@roboGod_goSuccessfulLargeArea);
robotgame_main(team1,team2);

