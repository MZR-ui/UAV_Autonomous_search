# SkillState.msg —— bt_engine 技能状态机状态（离线分析对齐用）
Header header

# 状态枚举（与 skill_executor 的 State 一致）
uint8 IDLE=0
uint8 SPOOL_UP=1
uint8 TAKEOFF=2
uint8 HOVER=3
uint8 TRAJECTORY=4
uint8 LANDING=5

uint8 state          # 技能状态机当前状态（见上枚举）
bool  armed          # 飞控已解锁（/msp/status armingFlags bit2）
bool  auto_mode      # CH8>=1350 自动模式；false=手动直通（此时 state 无意义）
