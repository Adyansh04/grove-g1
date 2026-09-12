# A pixel the caller says belongs to one object, in the frame the grounder was looking at.
#
# What it is for: a detector asked for "the cup" cannot tell which cup, and a model that can
# reason about "the one left of the bowl" can point at it. Backends that take points as an input
# use it directly; the rest use it to pick which of their own boxes to keep.

string phrase
uint32 x
uint32 y
