import pytest

def map_value(x, in_min, in_max, out_min, out_max):
    # ... 同上 ...
    """线性映射函数"""
    if x >= in_max:
        return out_max
    if x <= in_min:
        return out_min
    if in_min > in_max:
        return map_value(x, in_max, in_min, out_max, out_min)
    if out_min == out_max:
        return out_min
    
    in_mid = (in_min + in_max) >> 1
    out_mid = (out_min + out_max) >> 1
    
    if in_min == in_mid:
        return out_mid
    if x <= in_mid:
        return map_value(x, in_min, in_mid, out_min, out_mid)
    else:
        return map_value(x, in_mid + 1, in_max, out_mid, out_max)

        
class TestMapFunction:
    def test_boundaries(self):
        assert map_value(0, 0, 100, 0, 1000) == 0
        assert map_value(100, 0, 100, 0, 1000) == 1000
    
    def test_middle_value(self):
        assert map_value(50, 0, 100, 0, 1000) == 500
    
    def test_out_of_range(self):
        assert map_value(-10, 0, 100, 0, 1000) == 0
        assert map_value(150, 0, 100, 0, 1000) == 1000
    
    def test_reversed_input(self):
        assert map_value(50, 100, 0, 0, 1000) == 500
    
    def test_reversed_output(self):
        assert map_value(0, 0, 100, 1000, 0) == 1000
        assert map_value(100, 0, 100, 1000, 0) == 0
    
    def test_same_output_range(self):
        assert map_value(50, 0, 100, 500, 500) == 500
    
    def test_negative_ranges(self):
        assert map_value(-50, -100, 0, 0, 1000) == 500
        assert map_value(0, -100, 100, -1000, 1000) == 0

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
