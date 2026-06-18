# 上传本地代码到远程 GitHub 仓库教程

## 目录

- [1. 前置条件](#1-前置条件)
- [2. 完整流程](#2-完整流程)
- [3. 命令详解](#3-命令详解)
- [4. 常见问题](#4-常见问题)

---

## 1. 前置条件

- Git 已安装（`git --version` 验证）
- 已配置用户信息：

```bash
git config --global user.name "你的用户名"
git config --global user.email "你的邮箱@example.com"
```

- 已在 GitHub 上创建好一个空仓库（**不要勾选** "Add a README" / ".gitignore" / "License"）

---

## 2. 完整流程

```bash
# 1. 进入项目目录
cd /path/to/your/project

# 2. 初始化 Git 仓库（如果还没有 .git 目录）
git init

# 3. 查看文件状态
git status

# 4. 添加所有文件到暂存区（staging area）
git add .

# 5. 也可以选择性添加
git add src/main.py include/header.h

# 6. 提交代码到本地仓库
git commit -m "Initial commit"

# 7. 添加远程仓库地址
git remote add origin https://github.com/你的用户名/你的仓库名.git

# 8. 推送到 GitHub
git push -u origin master
# 或 git push -u origin main（如果默认分支是 main）
```

---

## 3. 命令详解

### `git init`

在当前目录创建一个 `.git` 隐藏目录，包含版本控制所需的所有元数据。

```bash
git init
```

> 如果目录已有 `.git`，会显示 `Reinitialized existing Git repository`，不会覆盖已有内容。

---

### `git status`

查看当前仓库状态：哪些文件被修改、哪些已暂存、哪些未跟踪。

```bash
git status
```

常用缩写：

```bash
git status -s   # 简洁模式
```

输出示例：

```
 M README.md       # 已修改但未暂存
A  newfile.txt     # 已暂存（新文件）
?? untracked.py    # 未跟踪的新文件
```

---

### `git add`

把文件从工作区加入暂存区（staging area），准备提交。

```bash
git add .                    # 添加所有变更（新文件 + 修改 + 删除）
git add *.py                 # 添加所有 .py 文件
git add src/                 # 添加 src/ 目录下所有内容
git add -A                   # 同 git add .（含已删除文件的记录）
git add -u                   # 只添加已跟踪文件的修改（不添加新文件）
```

---

### `git commit`

把暂存区的内容保存为一个永久的快照（commit）。

```bash
git commit -m "提交说明"           # 单行提交信息
git commit                          # 会打开编辑器写多行信息
git commit -am "说明"               # 跳过 git add，直接提交已跟踪文件的修改
```

> `-a` 只对 **已跟踪** 文件有效（即之前至少被 `git add` 过一次的文件），新文件仍需 `git add`。

---

### `git remote add`

关联一个远程仓库，`origin` 是远程仓库的默认别名。

```bash
git remote add origin https://github.com/用户名/仓库名.git
```

查看当前关联的远程仓库：

```bash
git remote -v
```

输出：

```
origin	https://github.com/用户名/仓库名.git (fetch)
origin	https://github.com/用户名/仓库名.git (push)
```

删除远程关联：

```bash
git remote remove origin
```

---

### `git push`

把本地 commit 上传到远程仓库。

```bash
git push -u origin master           # 首次推送，并关联上游分支
git push                            # 之后只需 git push
git push origin main                # 推送到 main 分支
git push --force origin master      # 强制推送（谨慎使用！会覆盖远程历史）
```

> `-u`（`--set-upstream`）建立本地分支与远程分支的关联，之后只需 `git push` 即可。

---

### `git log`

查看提交历史：

```bash
git log                    # 完整日志
git log --oneline          # 简洁模式
git log --graph --oneline  # 带分支图
```

---

## 4. 常见问题

### 4.1 `remote origin already exists`

远程地址已存在，先删除再重新添加：

```bash
git remote remove origin
git remote add origin https://github.com/用户名/仓库名.git
```

或直接修改地址：

```bash
git remote set-url origin https://github.com/用户名/仓库名.git
```

---

### 4.2 默认分支是 `main` 而不是 `master`

GitHub 新仓库默认分支为 `main`，而本地 `git init` 默认分支名为 `master`。

推送到 master 后 GitHub 会提示；或在推送前改名：

```bash
git branch -M main         # 把当前分支重命名为 main
git push -u origin main
```

---

### 4.3 `.gitattributes` 格式错误

`.gitattributes` 不是 shell 脚本，不要用 `cat > file << 'EOF'` 语法。正确的格式：

```
* text=auto eol=lf
*.png binary
*.jpg binary
```

每行一个规则，不含 shell 命令。

---

### 4.4 `.gitignore` 忽略不需要的文件

创建 `.gitignore` 防止编译产物、缓存、临时文件被提交：

```
# Python
__pycache__/
*.pyc
*.pyo

# C++ 编译
build/
*.o
*.a
*.so

# IDE
.vscode/
.idea/

# 系统
.DS_Store
Thumbs.db
```

---

### 4.5 认证失败（身份验证）

GitHub 已不支持密码认证，推荐以下方式之一：

**方式一：Personal Access Token（推荐）**

1. GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. 生成 token（勾选 `repo` 权限）
3. 推送时用 token 代替密码

**方式二：SSH**

```bash
# 生成 SSH 密钥（如果还没有）
ssh-keygen -t ed25519 -C "your_email@example.com"

# 添加到 ssh-agent
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# 把 ~/.ssh/id_ed25519.pub 的内容添加到 GitHub → Settings → SSH and GPG keys
```

然后把远程地址改为 SSH 格式：

```bash
git remote set-url origin git@github.com:用户名/仓库名.git
```

---

### 4.6 推送被拒绝（`non-fast-forward`）

远程已有本地没有的 commit，先拉取再推送：

```bash
git pull origin master --rebase
git push origin master
```

---

### 4.7 `.gitattributes:1 is not a valid attribute name`

文件第一行有非法语法。例如用 shell heredoc（`cat > file << 'EOF'`）创建的文件会包含命令本身。重新写入正确内容即可。

---

## 5. 速查卡片

| 场景 | 命令 |
|------|------|
| 初始化仓库 | `git init` |
| 查看状态 | `git status` |
| 添加所有文件 | `git add .` |
| 提交 | `git commit -m "信息"` |
| 关联远程 | `git remote add origin <URL>` |
| 首次推送 | `git push -u origin master` |
| 后续推送 | `git push` |
| 查看远程 | `git remote -v` |
| 查看日志 | `git log --oneline --graph` |
