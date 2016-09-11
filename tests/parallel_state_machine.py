#!/usr/bin/env python

import time

import smach


class Foo(smach.State):
    def __init__(self, name):
        smach.State.__init__(
            self, outcomes=['done', 'error'])
        self.name = name

    def execute(self, userdata):
        for i in xrange(10):
            print("%s %i" % (self.name, i))
            time.sleep(0.1)
        return 'done'


if __name__ == '__main__':
    sm = smach.Concurrence(
        outcomes=['error', 'done'],
        default_outcome='error',
        outcome_map={'done': {'Foo': 'done', 'Bar': 'done'}})

    with sm:
        smach.Concurrence.add('Foo', Foo('Foo'))
        smach.Concurrence.add('Bar', Foo('Bar'))

    outcome = sm.execute()
    print("Finished with: %s" % outcome)
