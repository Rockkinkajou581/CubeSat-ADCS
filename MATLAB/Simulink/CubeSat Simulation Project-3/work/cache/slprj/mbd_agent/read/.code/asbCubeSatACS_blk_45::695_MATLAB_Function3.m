function prov_eci = eceftoeci(prov_ecef, JD)
    % Step 1: Unix time -> Julian Date

    % Step 2: Julian Date -> GMST (degrees)
    theta_deg = jd_to_gmst(JD);

    % Step 3: Build rotation matrix R_z(theta)
    theta = deg2rad(theta_deg);
    R = [ cos(theta), -sin(theta), 0; ...
          sin(theta),  cos(theta), 0; ...
          0,           0,          1];

    % Step 4: Rotate position
    prov_eci = R * prov_ecef;

end


% -------------------------------------------------------------------------
function theta_deg = jd_to_gmst(JD)
% JD_TO_GMST Compute Greenwich Mean Sidereal Time (degrees) from Julian Date.

    % Julian centuries since J2000.0
    T = (JD - 2451545.0) / 36525.0;

    % GMST in degrees
    theta_deg = 280.46061837 ...
              + 360.98564736629 * (JD - 2451545.0) ...
              + 0.000387933     * T^2 ...
              - T^3             / 38710000.0;

    % Reduce to [0, 360)
    theta_deg = mod(theta_deg, 360.0);
end
