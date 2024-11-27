import pstats
from pstats import SortKey

p = pstats.Stats("restats2")
# p.strip_dirs().sort_stats(SortKey.CUMULATIVE).print_stats(30)
# p.sort_stats(SortKey.FILENAME).print_stats("set_robot_cell")
# p.strip_dirs().print_callers(0.5, "init")
p.strip_dirs().sort_stats(SortKey.CUMULATIVE).print_callees(10)
