import torch
import torch.nn.functional as F

def distillation_loss(student_logits, teacher_logits, temperature=2.0):
    student_soft = F.log_softmax(student_logits / temperature, dim=1)
    teacher_soft = F.softmax(teacher_logits / temperature, dim=1)
    loss = F.kl_div(student_soft, teacher_soft, reduction='batchmean') * (temperature ** 2)
    return loss


def combined_loss(student_logits, teacher_logits, targets, alpha=0.5, temperature=2.0):
    ce_loss = F.cross_entropy(student_logits, targets)
    kd_loss = distillation_loss(student_logits, teacher_logits, temperature)
    loss = alpha * kd_loss + (1 - alpha) * ce_loss
    return loss



if __name__ == "__main__":
    student_logits = torch.randn(4, 10, requires_grad=True)
    teacher_logits = torch.randn(4, 10)
    targets = torch.tensor([1, 2, 3, 4])

    loss = combined_loss(student_logits, teacher_logits, targets)
    print(f"Combined loss: {loss.item()}")
