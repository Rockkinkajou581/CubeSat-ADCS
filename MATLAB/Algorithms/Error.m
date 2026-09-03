function [deg_error, w_error] = fcn(q_est, q_true, w_est, w_true)
    q_error = quatmultiply(quatinv(q_est), q_true);
    if (q_error(1) < 0)
        q_error = -q_error;
    end
    deg_error = rad2deg(2 * acos(q_error(1)));
    w_error = norm(w_est - w_true);
end