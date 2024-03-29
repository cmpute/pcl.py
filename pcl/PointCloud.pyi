from pcl.PointField import PointField
from typing import Any, Dict, Union, NewType, List, Iterator, Optional
from numpy import ndarray, dtype

ros_PointCloud2 = NewType("sensor_msgs.PointCloud2", object)

class PointCloud:
    def __init__(self, data: Union[List[tuple], ndarray, ros_PointCloud2, PointCloud] = None, point_type: str = 'XYZ') -> None: ...
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def fields(self) -> List[PointField]: ...
    @property
    def names(self) -> List[str]: ...
    @property
    def sensor_orientation(self) -> ndarray: ...
    @property
    def sensor_origin(self) -> ndarray: ...
    @property
    def xyz(self) -> ndarray: ...
    @property
    def normal(self) -> ndarray: ...
    @property
    def rgb(self) -> ndarray: ...
    @property
    def argb(self) -> ndarray: ...
    @property
    def is_organized(self) -> bool: ...
    @property
    def ptype(self) -> str: ...
    @property
    def nptype(self) -> dtype: ...
    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...
    def __reduce__(self) -> tuple: ...
    def __iter__(self) -> Iterator[ndarray]: ...
    def __contains__(self, item) -> bool: ...
    def __getitem__(self, indices: Union[ndarray, int, slice, str, List[str]]) -> PointCloud: ...
    def __setitem__(self, indices: Union[ndarray, int, slice, str, List[str]], value: Any): ...
    def append(self, points: Union[ndarray, PointCloud]) -> PointCloud: ...
    def insert(self, points: Union[ndarray, PointCloud], offsets: int) -> PointCloud: ...
    def append_fields(self, data: Optional[Union[ndarray, PointCloud, Dict[str, ndarray]]]=None, **kwargs) -> PointCloud: ...
    def insert_fields(self, data: Union[ndarray, PointCloud], offsets: Union[Dict[str, int], List[int]]) -> PointCloud: ...
    def __add__(self, item: Union[ndarray, PointCloud]): ...
    def __eq__(self, target: PointCloud): ...
    def __ne__(self, target: PointCloud): ...
    def to_ndarray(self, fields: List[str] = None) -> ndarray: ...
    def to_msg(self) -> ros_PointCloud2: ...
    def infer_ptype(self) -> None: ...
