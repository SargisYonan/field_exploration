clc;
clear all;
close all;

field_dir = 'fields/';
field_size = 20;
sigma_fields = [4];
seed = 3;

addpath(genpath(pwd))
rng(seed);

for sfix = 1 : length(sigma_fields)
    field = Field(field_size, field_size, sigma_fields(sfix));
    save(strcat([field_dir, 'field_', num2str(field_size), 'x', num2str(field_size), ...
    '_sf_', num2str(sigma_fields(sfix)), '_seed_', num2str(seed)]), ...
    'field');
end