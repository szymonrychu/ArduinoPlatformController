
from uuid import UUID
from .message_utils import MoveRequest, MoveStatus, StopRequest

class MoveRegistry():

    def __init__(self):
        self._move_registry = {}
        self._current_move_uuid = None
        self._current_move_progress = 0
        self._current_move_part = 0
        self._current_move_parts_num = 0

    def _check_if_move_in_registry(self, uuid:UUID) -> bool:
        return uuid in self._move_registry.keys()

    def request_move(self, m:MoveRequest) -> str:
        self._move_registry[m.uuid] = m

    def get_next_move(self) -> MoveRequest:
        return next(iter(self._move_registry))

    def handle_moves_update(self, status:MoveStatus):
        if status:
            # we are processing non-None move status, therefore we're moving
            if self._current_move_uuid != status.uuid:
                # we are tracking outdated move, so we have to drop it from registry
                # and update the current_move_uuid
                self.drop_move_from_registry(self._current_move_uuid)
                self._current_move_uuid = status.uuid
            
            if not self._check_if_move_in_registry(status.uuid):
                rospy.logerr(f"Move with UUID:{status.uuid} missing in registry!")
                return
            
            self._current_move_progress = status.progress
            self._current_move_part = status.part
            self._current_move_parts_num = status.max_parts

        else:
            # move status update was None, so we are not processing any movement now
            self.drop_move_from_registry(self._current_move_uuid)
            self._current_move_uuid = None

    def drop_move_from_registry(self, uuid:UUID) -> bool:
        try:
            del self._move_registry[uuid]
            return True
        except KeyError as _key_error:
            return False

    @property
    def current_move_uuid(self) -> UUID:
        return self._current_move_uuid

    @property
    def current_move_progress(self) -> float:
        return self._current_move_progress

    @property
    def current_move_part(self) -> float:
        return self._current_move_part

    @property
    def current_move_parts_num(self) -> float:
        return self._current_move_parts_num

        