# -*- coding: utf-8 -*-
"""
Created on Mon Apr 26 18:06:31 2021

@author: Timothe
"""

#TESTING SPEED OF REFACTORED LIST COMPREHENSIONS FROM FOR LOOPS

def FORMODE():
    my_numbers = [1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5]
    for i in range(800000):
        odd_numbers = []
        for item in my_numbers:
          if item % 2 == 1:
            odd_numbers.append(item)

def COMPREHENSIONMODE():
    my_numbers = [1,2,3,4,5,1,2,3,4,5,1,2,3,4,5,1,2,3,4,5]
    for i in range(800000):
        odd_numbers = [item for item in my_numbers if item % 2 == 1]


FORMODE()
COMPREHENSIONMODE()