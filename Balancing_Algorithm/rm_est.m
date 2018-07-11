function [r_mmu] = rm_est(r_hat, m, m_mmu)
%rm_est: CSACS, estimates the distance to move the stepper motors to remove the CM offset

% Used by CSACS_Sim

r_mmu = -m/m_mmu*r_hat;