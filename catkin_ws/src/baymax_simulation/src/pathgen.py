def squared_velocity_path(lamda, lamda_max, pos_delta):
    c = pos_delta*3/(2*(lamda_max/2)**3)
    if lamda <= lamda_max/2:
        return c*lamda**2
    else:
        return c*(lamda-lamda_max)**2