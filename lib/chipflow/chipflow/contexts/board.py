
class BoardContext():
    def __init__(self, platform):
        self.platform = platform

    def build(self):
        self.platform.build()
