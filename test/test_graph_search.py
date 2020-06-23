import pytest

from src.prechecks.graph_search import calculate_node_idx, joint_limit_cost


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
            calculate_node_idx(pt_idx, conf)


@pytest.mark.parametrize("joints,qlim,weights,cost,exc",
                         [
                             ([0], [-10, 10], None, 0, None),
                             ([0], [-10, 10], [2], 0, None),
                             ([0], [-10, 10], [2, 3], 0, ValueError),
                             ([5], [-10, 10], [2], 1 / 16, None),
                         ]
                         )
def test_joint_limit_distance(joints, qlim, weights, cost, exc):
    if exc is None:
        actual_cost = joint_limit_cost(joints, qlim, weights)
        assert actual_cost >= 0
        assert actual_cost == cost
    else:
        with pytest.raises(exc):
            joint_limit_cost(joints, qlim, weights)
