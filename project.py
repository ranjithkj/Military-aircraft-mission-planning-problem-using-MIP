#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 6 04:58:15 2017

@author: ranjith and stefana

Project : Vehicle Routing Model for the Military Aircraft Mission Planning Problem

Objective : To maximize the total expected effect
"""

import sys
import math
from random import randint
from gurobipy import *

# nodes - 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16

# (1,2) - Target 1 Sector 1 Attacking nodes

# (3,4) - Target 1 Sector 1 Illuminating nodes  

# (5,6) - Target 1 Sector 2 Attacking nodes

# (7,8) - Target 1 Sector 2 Illuminating nodes  

# (9,10) -  Target 2 Sector 1 Attacking nodes

# (11,12) - Target 2 Sector 1 Illuminating nodes 

# (13,14) -  Target 2 Sector 2 Attacking nodes  

# (15,16) -  Target 2 Sector 2 Illuminating nodes    

# Data
# A_g = [ {set of arcs for sector 1},{set of arcs for sector 2},set of arcs for sector 2 target 2           ]    


R = 2
M = 2
N = 16
G = [[1,2],[3,4]]
G_m = 4
T = [0,3,3]
q_m = [1,1]
u = 0.08
a = 240
A = 240
A_g = []
I_g = []

N_A_m = [ [1,2,3,4],[9,10,11,12] ]
N_I_m = [ [5,6,7,8],[13,14,15,16] ]

def time_gen(N):   # function to generate time points
 G = []    
 for i in range(1,N):
   G += [i]
 return G    

W = []  
W += [0]
W += time_gen(100)     
S = time_gen(100)

P = []

o = 20
d = N+1
E = {}

def assign_A(A,N):  # function to generate arcs
 
 J = N+2
 H = {}
 a = 1
 for i in range(J):
    for j in range(J):
       if i != j:
           if len(H) >= A:
               a = 1
               #do nothing
           else:
                H[i,j] = randint(1,3)
 

 #raw_input()          
 return H

A = assign_A(A,N)
C = A
S_ij = assign_A(a,N)
def assign_Ag(A,k):  # function to assign A_g
  
        F = {}  
        H = []  

        if k == 1:
            
            for (i,l) in A:
                if (l == 1):
                  F[i,1] = A[i,1]
            for (i,l) in A:
                if (l == 2):
                  F[i,2] = A[i,2]
            H.append(F)
            F = {}     
            return H
        if k == 2:

            for (i,l) in A:
                if (l == 3):
                  F[i,3] = A[i,3]
            for (i,l) in A:
                if (l == 4):
                  F[i,4] = A[i,4]
         
            H.append(F)  
            F = {}
            return H 
        if (k == 3):
        
            for (i,l) in A:
                if (l == 9):
                  F[i,9] = A[i,9]
            for (i,l) in A:
                if (l == 10):
                 F[i,10] = A[i,10]
            H.append(F)
            F = {}
            return H 
        if k == 4:
            for (i,l) in A:
                if (l == 11):
                  F[i,11] = A[i,11]
            for (i,l) in A:
                if (l == 12):
                  F[i,12] = A[i,12]
            H.append(F)        
            F = {}
            return H 
        return H

def assign_Ig(A,k):   # function to assign I_g
 
        F = {}  
        H = []  

        if k == 1:
            
            for (i,l) in A:
                if (l == 5):
                  F[i,5] = A[i,5]
            for (i,l) in A:
                if (l == 6):
                  F[i,6] = A[i,6]
            H.append(F)
            F = {}     
            return H
        if k == 2:

            for (i,l) in A:
                if (l == 7):
                  F[i,7] = A[i,7]
            for (i,l) in A:
                if (l == 8):
                  F[i,8] = A[i,8]
         
            H.append(F)  
            F = {}
            return H 
        if (k == 3):
        
            for (i,l) in A:
                if (l == 13):
                  F[i,13] = A[i,13]
            for (i,l) in A:
                if (l == 14):
                 F[i,14] = A[i,14]
            H.append(F)
            F = {}
            return H 
        if k == 4:
            for (i,l) in A:
                if (l == 15):
                  F[i,15] = A[i,15]
            for (i,l) in A:
                if (l == 16):
                  F[i,16] = A[i,16]
            H.append(F)        
            F = {}
            return H 
        return H

for i in range(1,G[len(G)-1][len(G[len(G)-1])-1]+1):

    A_g.append(assign_Ag(A, i))
    I_g.append(assign_Ig(A, i))


#Create Model
m = Model()


# Variables: aircraft traversing 

x = {}
for r in range(1,R+1):
   for (i,j) in A:
         x[r,i,j] = m.addVar(vtype=GRB.BINARY, name="x_"+str(r)+","+str(i)+","+str(j))
      
        
y = {}
for r in range(1,R+1):
  for i in range(0,N+2):
    for s in range(len(W)):
        y[r,i,s] = m.addVar(vtype=GRB.BINARY, name="y_"+str(r)+","+str(i)+","+str(s))
        
tend = m.addVar(vtype=GRB.CONTINUOUS, name="tend_")        
          
#constraints:

    
#Constraint - 2.1
for r in range(1,R+1):    
  m.addConstr( quicksum(x[r,0,j] for j in range(0,N+2) if (0,j) in A) == 1 )  #correct


#Constraint - 2.2
for r in range(1,R+1):  
  m.addConstr( quicksum(x[r,j,d] for j in range(0,N+2) if (j,d) in A) == 1 )   #correct


#Constraint - 2.3

sum1 = 0
sum2 = 0
for k in range(1,N+1):
  for r in range(1,R+1):
    sum1 = 0
    sum2 = 0
    sum1 = quicksum(x[r,i,k] for i in range(N+2) if (i,k) in A)
    sum2 = quicksum(x[r,k,j] for j in range(N+2) if (k,j) in A)
    m.addConstr(sum1 == sum2) 
  

#Constraint - 2.4
for p in range(0,M): 

  m.addConstr(quicksum(x[r,i,j] for i in range(N+2) for j in range(N+2) for r in range(1,R+1) for q in G[p] if (i,j) in A_g[q-1][0]) == 1)
           

#Constraint - 2.5
for p in range(M): 

  m.addConstr(quicksum(x[r,i,j] for i in range(N+2) for j in range(N+2) for r in range(1,R+1) for q in G[p] if (i,j) in I_g[q-1][0]) == 1)


#Constraint - 2.6
#2.6 -> correct

sum1 = 0 
sum2 = 0
for g in range(G_m): 
    sum1 = 0 
    sum2 = 0
    for r in range(1,R+1):  
      sum1 += quicksum(x[r,i,j] for i in range(N+2) for j in range(N+2) if (i,j) in A_g[g][0]) 
      sum2 += quicksum(x[r,i,j] for i in range(N+2) for j in range(N+2) if (i,j) in I_g[g][0])
    m.addConstr(sum1 == sum2)   

#Constraint - 2.7         
#2.7.1   

sum1 = 0
for p in range(M):
  for r in range(1,R+1):
      sum1 = 0
      for g in G[p]:
           J = dict(A_g[g-1][0].items()[:] + I_g[g-1][0].items()[:]) 
           sum1 += quicksum(x[r,i,j] for (i,j) in J)
      m.addConstr(sum1 <= 1)

#Constraint - 2.8   
#2.8 
         
for r in range(1,R+1):
    m.addConstr(quicksum(x[r,i,j] * q_m[p] for p in range(M) for i in range(N+2) for j in range(N+2) for q in G[p] if (i,j) in A_g[q-1][0]) <= T[r])
            
#Constraint - 2.9           
#2.9

for r in range(1,R+1): 
  m.addConstr(y[r,0,0] == 1) 

 
#Constraint - 2.10
#2.10


sum1 = 0
for r in range(1,R+1):  
    for (i,j) in A:
       for w in W: 
           sum1 = 0
           sum2 = x[r,i,j] + y[r,i,w] - 1
           sum1 += quicksum(y[r,j,t] for t in range(w + S_ij[i,j],len(S)+1) if (i,j) in A)
           m.addConstr(sum1 >= sum2)          



#Constraint - 2.11
#2.11

sum1 = 0
sum2 = 0
for r in range(1,R+1):
    for k in range(1,N+1):
        
        sum1 = quicksum(y[r,k,s] for s in S if (k,j) in A)
        sum2 = quicksum(x[r,k,j] for j in range(N+2) if (k,j) in A)      
        m.addConstr( sum1 == sum2 )

#Constraint - 2.12
 
#2.12

for p in range(M):
   for s in S:
      sum1 = 0
      sum2 = 0
      sum1 = quicksum(y[r,i,s] for i in N_A_m[p] for r in range(1,R+1))
      sum2 = quicksum(y[r,j,s] for j in N_I_m[p] for r in range(1,R+1))
      m.addConstr(sum1 == sum2)

# 2.13

for p in P:
   for n in P:
      if ( p!= n):
        for s in S:
          m.addConstr(quicksum(y[r,i,t] for r in range(1,R+1) for t in S for i in N_A_m[p-1]) >= quicksum(y[r,i,s] for r in range(1,R+1) for i in N_I_m[n-1]))

# 2.14

for p in P:
   for n in P:
      if ( p!= n):
       for r in range(1,R+1):   
         for s in S:
          m.addConstr(quicksum(y[r,i,t] for t in range(1,s) for i in N_A_m[p-1]) + quicksum(y[r,i,s] for i in N_I_m[n-1]) <= 1)


#Constraint - 2.15
#2.15


for r in range(1,R+1):
   for i in range(0,N+2):  # and 0,d
     m.addConstr(quicksum(y[r,i,s] for s in S) <= 1)


#Constraint - 2.16     
#2.16

for r in range(1,R+1):
    m.addConstr(quicksum(y[r,d,s]*s for s in W) <= tend)
 


obj = 0
for r in range(1,R+1):
   for (i,j) in A:

     obj += C[i,j]*x[r,i,j]
          
          
obj = obj + u*tend 


m.write("model.lp")


m.params.timelimit = 300  

# Create Objective Function

m.setObjective(obj, GRB.MINIMIZE)
   
m.optimize()


for r in range(1,R+1):
   for (i,j) in A:
    if x[r,i,j].x == 1:
      print r,i,j
          

print "y values"

for r in range(1,R+1):
  for i in range(0,N+2):
    for s in range(len(W)):
            if y[r,i,s].x == 1:
                print r,i,s
 
  
print tend.x    