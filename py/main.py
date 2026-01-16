from pathlib import Path
from datetime import datetime


p = Path('../../../../michealellis/Downloads')
build = list(p.glob('**/*'))

for i in build:
    if not i.is_file():
        continue
    suffix = i.suffix          
    created_dt = datetime.fromtimestamp(i.stat().st_ctime)
    created_date = created_dt.strftime("%Y-%m")
    nested_path = Path(f"{p}/{suffix.lstrip('.')}/{created_date}")
    nested_path.mkdir(parents=True, exist_ok=True)
    destination = Path(f"{nested_path}/{i.name}")
    i.rename(destination)

print('Migration completed')   
        
    

