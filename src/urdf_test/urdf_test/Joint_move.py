class Joint_move:
  def __init__(self):
    self.height = 0.
    self.hinc = 0.02
    self.rotation = 0.
    self.go_up = True

  def calc_height(self):
    if self.height < 0.3:
      self.go_up = True
    elif self.height > 2.0:
      self.go_up = False

    if self.go_up:
      self.height += self.hinc
    else:
      self.height -= self.hinc

    return self.get_height()

  def calc_rotation(self):
    self.rotation += 0.1
    return self.get_rotation()

  def get_rotation(self):
    return self.rotation

  def get_height(self):
    return self.height    
