function [robot, arena, obstacle, endPts] = loadPlanningScene(dim, configPath)
robot = load([configPath, 'robot_config_', dim, '.csv']);

obstacle = load([configPath, 'obstacle_config_', dim ,'.csv']);
arena = load([configPath, 'arena_config_', dim ,'.csv']);

endPts = load([configPath, 'end_points_', dim, '.csv']);