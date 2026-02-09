class DefaultDict:
    """
    Custom default dict class.
    """

    def __init__(self, default_item=0):
        self.dict = {}
        self.default_item = default_item

    def __setitem__(self, key, value):
        self.dict[key] = value

    def __getitem__(self, key):
        if key not in self.dict.keys():
            return self.default_item
        else:
            return self.dict[key]

    def __contains__(self, key):
        return key in self.dict

    def pop(self, key):
        self.dict.pop(key)

    def __repr__(self):
        return repr(self.dict)
