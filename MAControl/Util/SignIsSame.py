
def sign_is_same(a, b):

    a_is_positive = False
    b_is_positive = False

    if a > 0:
        a_is_positive = True
    if b > 0:
        b_is_positive = True

    if a_is_positive is b_is_positive:
        return True
    else:
        return False
