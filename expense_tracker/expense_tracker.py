"""
Expense Tracker Module
----------------------
Manages collections of expenses with functionality for adding, filtering,
analyzing, and persisting expense data to JSON files.
"""

from expense import Expense
import json
from datetime import datetime


class ExpenseTracker:
    """
    Manages a collection of Expense objects with persistence and analysis capabilities.
    
    Attributes:
        expenses (list): List of Expense objects being tracked
    """
    
    def __init__(self):
        """Initialize an empty ExpenseTracker."""
        self.expenses = []
    
    def add_expense(self, expense_object):
        """
        Add an expense to the tracker.
        
        Args:
            expense_object (Expense): The expense to add
        """
        self.expenses.append(expense_object)
    
    def show_all(self):
        """Print all expenses in the tracker, one per line."""
        for expense in self.expenses:
            print(expense)
    
    def total_spent(self):
        """
        Calculate the total amount spent across all expenses.
        
        Returns:
            float: Sum of all expense amounts
        """
        total = 0
        for expense in self.expenses:
            total += expense.amount
        return total
    
    def cash_expenses(self):
        """
        Filter expenses to only those paid with cash.
        
        Returns:
            list: List of Expense objects paid with cash
        """
        return [expense for expense in self.expenses if expense.is_cash()]
    
    def save_to_file(self, filename):
        """
        Save all expenses to a JSON file.
        
        Args:
            filename (str): Path to the JSON file to write
        """
        expenses_list = [
            {
                "amount": expense.amount,
                "category": expense.category,
                "description": expense.description,
                "payment_method": expense.payment_method,
                "date": expense.date
            }
            for expense in self.expenses
        ]
        
        with open(filename, 'w') as file:
            json.dump(expenses_list, file)
    
    def load_from_file(self, filename):
        """
        Load expenses from a JSON file and add them to the tracker.
        
        Args:
            filename (str): Path to the JSON file to read
        
        Prints:
            Success message with count of loaded expenses, or error message if file
            is not found or contains invalid JSON
        """
        try:
            with open(filename, 'r') as file:
                file_content = json.load(file)
                
                for expense_dict in file_content:
                    # Extract fields from dictionary
                    amount = expense_dict["amount"]
                    category = expense_dict["category"]
                    description = expense_dict["description"]
                    payment_method = expense_dict["payment_method"]
                    date = expense_dict["date"]
                    
                    # Create Expense object and add to tracker
                    expense = Expense(amount, category, description, payment_method, date)
                    self.expenses.append(expense)
                    
                print(f"Loaded {len(self.expenses)} expenses")
                
        except FileNotFoundError:
            print('File not found')
        except json.JSONDecodeError:
            print('File is corrupted or invalid JSON')
    
    def expenses_in_range(self, start_date, end_date):
        """
        Filter expenses by date range (inclusive).
        
        Args:
            start_date (str): Start date in YYYY-MM-DD format
            end_date (str): End date in YYYY-MM-DD format
        
        Returns:
            list: Expense objects within the date range
        """
        # Convert string dates to date objects for proper comparison
        start = datetime.strptime(start_date, "%Y-%m-%d").date()
        end = datetime.strptime(end_date, "%Y-%m-%d").date()
        
        return [
            expense for expense in self.expenses 
            if datetime.strptime(expense.date, "%Y-%m-%d").date() >= start
            and datetime.strptime(expense.date, "%Y-%m-%d").date() <= end
        ]
    
    def spending_by_category(self):
        """
        Calculate total spending grouped by category.
        
        Returns:
            dict: Dictionary mapping category names (str) to total amounts (float)
        """
        summary = {}
        for expense in self.expenses:
            category = expense.category
            if category in summary:
                summary[category] += expense.amount
            else:
                summary[category] = expense.amount
        return summary


# Test code - only runs when file is executed directly
if __name__ == '__main__':
    tracker = ExpenseTracker()

    # Add test expenses
    tracker.add_expense(Expense(45.00, "Groceries", "Weekly Shopping", "Cash", "2025-01-10"))
    tracker.add_expense(Expense(12.50, "Transportaion", "Uber to work", "Credit", "2025-01-11"))
    tracker.add_expense(Expense(50.00, "Subscriptions", "Claude", "Credit", "2025-01-12"))

    # Display all expenses
    tracker.show_all()
    print(f"\nTotal spent: ${tracker.total_spent()}")

    # Filter cash expenses
    print("\nCash Expenses only:")
    cash_only = tracker.cash_expenses()
    for expense in cash_only:
        print(expense)
    
    # Test file persistence
    tracker.save_to_file("expenses.json")
    print("\nSaved to expenses.json!")

    new_tracker = ExpenseTracker()
    new_tracker.load_from_file("expenses.json")
    print("\nLoaded from file:")
    new_tracker.show_all()

    # Test date filtering
    print("\nExpenses from Jan 10-11:")
    filtered = tracker.expenses_in_range("2025-01-10", "2025-01-11")
    for expense in filtered:
        print(expense)
    
    # Test category summary
    print("\nSpending by category:")
    summary = tracker.spending_by_category()
    for category, total in summary.items():
        print(f"{category}: ${total}")