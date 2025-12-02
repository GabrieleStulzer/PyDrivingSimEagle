def areo_model(u, params):

    # ----------------------------------------------------------------
    ## Function purpose: compute aerodynamic drag and downforce on front 
    ##                   and rear axles
    # ----------------------------------------------------------------

    # Load vehicle aero data
    CAx  = params.vehicle.CAx
    CAzf = params.vehicle.CAzf
    CAzr = params.vehicle.CAzr
    CV = params.vehicle.cv

    FAxc   = CAx * u**2 + CV*u
    FAz__r = CAzr * u**2
    FAz__f = CAzf * u**2
    
    return FAxc, FAz__r, FAz__f