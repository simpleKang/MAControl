def constrain(x, Min, Max):
    if x < Min:
        return Min
    elif x > Max:
        return Max
    else:
        return x
