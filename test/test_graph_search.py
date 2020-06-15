import pytest

from src.prechecks.graph_search import calculate_node_idx


@pytest.mark.parametrize("pt_idx,conf,node_idx,exc",
                         [
                             (0, 0, 0, None),
                             (0, 1, 1, None),
                             (0, -1, None, ValueError),
                             (0, 8, None, ValueError),
                             (1, 1, 9, None),
                             (1, 7, 15, None),
                             (-1, 7, None, ValueError),
                         ]
                         )
def test_calculate_node_idx(pt_idx, conf, node_idx, exc):
    if exc is None:
        act_node_idx = calculate_node_idx(pt_idx, conf)
        assert act_node_idx == node_idx
    else:
        with pytest.raises(exc):
            act_node_idx = calculate_node_idx(pt_idx, conf)
