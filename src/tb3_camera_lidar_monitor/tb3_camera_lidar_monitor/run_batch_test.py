import sys
import os
import random
import csv

# 把当前目录加入路径，确保能导入 core
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from core import ConsistencyMonitor

def run_simulation(confirmations, num_trials=44):
    print(f"\n=== 开始批量测试：配置为 {confirmations} 次连续不一致触发 ===")
    
    success_count = 0
    total_attacks = 0
    
    # 准备写入 CSV 文件，方便你以后看数据
    csv_filename = f"batch_results_{confirmations}_times.csv"
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Trial", "Attack_Length", "Triggered_Alert", "Triggered_At_Count"])

        for i in range(num_trials):
            # 初始化监控器 (使用你的配置)
            monitor = ConsistencyMonitor(near=1.0, far=2.0, confirmations=confirmations)
            
            # 随机模拟一次攻击的持续长度（比如连续发生 1 到 8 次不一致）
            attack_length = random.randint(1, 8)
            triggered = False
            triggered_at = 0
            
            # 模拟注入攻击数据
            for step in range(attack_length):
                stamp = 100.0 + step * 1.0
                # 注入不一致：摄像头False(没看到)，雷达0.5(看到了)
                monitor.add('image', stamp, False)
                results = monitor.add('scan', stamp + 0.05, 0.5)
                
                if results and results[0]['alert']:
                    triggered = True
                    triggered_at = results[0]['count']
                    break # 已经触发了，本次测试结束
            
            # 统计结果
            if triggered:
                success_count += 1
            
            # 写入 CSV
            writer.writerow([i+1, attack_length, triggered, triggered_at])

    print(f"总计运行: {num_trials} 遍")
    print(f"成功触发报警次数: {success_count} 遍")
    print(f"结果已保存到: {csv_filename}")

if __name__ == '__main__':
    # 测试你的 2 次配置
    run_simulation(confirmations=2, num_trials=44)
    
    # 测试你的 5 次配置
    run_simulation(confirmations=5, num_trials=44)