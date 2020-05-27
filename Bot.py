import Procedure
import Strategy
import Handling

#This file is just an abstract layer for calling all the real logic

def Process(self, game, packet, info, lock):

    Procedure.pre_process(self, game, packet, lock)
    Procedure.rlu_pp(self, info)
    Strategy.plan(self)

    Handling.controls(self)

    Procedure.feedback(self)

    Procedure.finish(self)

    return Handling.output(self)
