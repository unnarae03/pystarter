import py_trees
import py_trees.behaviours
import py_trees.common

# === 사용자 정의 노드 ===

class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="MoveToGoal")

    def update(self):
        print("📍 MoveToGoal 실행")
        return py_trees.common.Status.SUCCESS

class SetAngle(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="SetAngle")

    def update(self):
        print("🌀 SetAngle 실행")
        return py_trees.common.Status.SUCCESS

class CaptureImage(py_trees.behaviour.Behaviour):
    def __init__(self, label):
        super().__init__(name=f"CaptureImage(label={label})")
        self.label = label

    def update(self):
        print(f"📸 CaptureImage 실행 (label={self.label})")
        return py_trees.common.Status.SUCCESS

class CheckImageMatch(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckImageMatch", simulate_success=False):
        super().__init__(name=name)
        self.simulate_success = simulate_success

    def update(self):
        result = py_trees.common.Status.SUCCESS if self.simulate_success else py_trees.common.Status.FAILURE
        print(f"🔍 {self.name} 실행 → {result.name}")
        return result

class AdjustAngleThreshold(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="AdjustAngleThreshold")

    def update(self):
        print("🔧 AdjustAngleThreshold 실행")
        return py_trees.common.Status.SUCCESS

class SaveResult(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="SaveResult")

    def update(self):
        print("💾 SaveResult 실행")
        return py_trees.common.Status.SUCCESS

# === 트리 생성 ===

def create_tree():
    root = py_trees.composites.Sequence(name="SingleExecution", memory=False)

    move_to_goal = MoveToGoal()
    set_angle = SetAngle()
    capture1 = CaptureImage(label=1)

    check1 = CheckImageMatch(name="CheckImageMatch1", simulate_success=False)  # 첫 시도 실패
    adjust_angle = AdjustAngleThreshold()
    capture2 = CaptureImage(label=2)
    check2 = CheckImageMatch(name="CheckImageMatch2", simulate_success=True)   # 두 번째 시도 성공

    retry_sequence = py_trees.composites.Sequence(name="RetrySequence", memory=False)
    retry_sequence.add_children([adjust_angle, capture2, check2])

    fallback = py_trees.composites.Selector(name="ImageCheckFallback", memory=False)
    fallback.add_children([check1, retry_sequence])

    save_result = SaveResult()

    root.add_children([move_to_goal, set_angle, capture1, fallback, save_result])
    return root

# === 실행 ===

if __name__ == "__main__":
    tree = py_trees.trees.BehaviourTree(create_tree())
    print("\n🚀 py_trees 트리 실행 시작...\n")
    tree.tick_tock(period_ms=1000, number_of_iterations=1)
    print("\n✅ py_trees 트리 실행 완료.\n")
