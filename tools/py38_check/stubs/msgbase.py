class M:
    def __init__(self,*a,**k): pass
    def __getattr__(self,n):
        if n.startswith('__'): raise AttributeError(n)
        v=M(); object.__setattr__(self,n,v); return v
def __getattr__(n):
    if n.startswith('__'): raise AttributeError(n)
    return type(n,(M,),{})
