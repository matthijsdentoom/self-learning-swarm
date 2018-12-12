
def clip(min_val, max_val, value):
    """Clips the value between the min_val and the max_val"""
    return max(min_val, min(max_val, value))
