# A pixel on one object, in the image the grounder looked at. A detector asked for "the cup"
# cannot tell which cup; this says which. Backends that take points use it directly, the rest
# use it to choose among their own boxes.

# The grounded phrase this point belongs to.
string phrase
# Pixel column and row.
uint32 x
uint32 y
